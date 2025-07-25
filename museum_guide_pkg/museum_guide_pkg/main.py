#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
스마트 전시 가이드 로봇 - 메인 실행 파일
팀 A: UI 및 시스템 통합 담당
"""

from museum_guide_pkg.ui_manager import MuseumGuideUI
from museum_guide_pkg.user_profile import UserProfile

import sys
import time
import signal
import os
import json
import shutil
from datetime import datetime
import subprocess

import ament_index_python
from ament_index_python.packages import get_package_share_directory

# ROS2 통합은 선택적으로 import
try:
    import rclpy
    from rclpy.node import Node
    from museum_guide_pkg.ros2_integration import ROS2IntegrationManager, load_user_data_from_json, convert_user_interests_to_our_system
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    # self.get_logger().warn("ROS2 통합 모듈 없음 - 기본 모드로 실행") # Suppress this print

def migrate_old_structure(node_logger):
    """기존 users/ 디렉토리를 profiles/로 통합"""
    script_dir = os.path.dirname(__file__)
    users_dir = os.path.join(script_dir, '..', 'users') # Assuming 'users' is at the package root level
    profiles_dir = os.path.join(script_dir, '..', 'profiles') # Assuming 'profiles' is at the package root level
   
    # profiles 디렉토리 생성
    if not os.path.exists(profiles_dir):
        os.makedirs(profiles_dir)
   
    # users/ 디렉토리가 존재하면 이동
    if os.path.exists(users_dir):
        node_logger.info("🔄 기존 users/ 디렉토리를 profiles/로 통합 중...")
       
        moved_count = 0
        for filename in os.listdir(users_dir):
            if filename.endswith('.json'):
                old_path = os.path.join(users_dir, filename)
                new_path = os.path.join(profiles_dir, filename)
               
                # 파일이 이미 존재하지 않을 때만 이동
                if not os.path.exists(new_path):
                    shutil.move(old_path, new_path)
                    node_logger.info(f"📁 이동: {filename}")
                    moved_count += 1
                else:
                    node_logger.warn(f"⚠️ 이미 존재함, 건너뛰기: {filename}")
       
        # 빈 users/ 디렉토리 제거
        try:
            if not os.listdir(users_dir):
                os.rmdir(users_dir)
                node_logger.info("🗑️ 빈 users/ 디렉토리 제거")
        except Exception as e:
            node_logger.error(f"빈 users/ 디렉토리 제거 실패: {e}")
       
        node_logger.info(f"✅ 통합 완료: {moved_count}개 파일 이동")
    else:
        node_logger.info("📂 users/ 디렉토리가 없습니다. 통합 작업 건너뛰기.")

class MuseumGuideSystem(Node):
    def __init__(self):
        """박물관 가이드 시스템 초기화"""
        super().__init__('museum_guide_node')
        self.get_logger().info("MuseumGuideSystem node initializing...")
        self.ui = MuseumGuideUI()
        self.user_profile = UserProfile()
        
        self.running = True
        self.ros_launch_process = None # ROS2 Launch 프로세스 핸들
       
        # 중복 프로필 정리 (최초 1회)
        self.user_profile.migrate_duplicate_profiles(self.get_logger())
       
        # ROS2 통합 매니저 초기화 (있는 경우에만)
        if ROS2_AVAILABLE:
            try:
                self.ros_manager = ROS2IntegrationManager(self.get_logger())
                self.get_logger().info("ROS2 통합 시스템 초기화 완료")
            except Exception as e:
                self.get_logger().error(f"ROS2 초기화 실패: {e}")
                self.ros_manager = None
        else:
            self.get_logger().warn("ROS2 환경이 아니므로 Launch 파일을 실행하지 않습니다.")
            self.ros_manager = None
       
        # JSON 파일에서 전시품 정보 로드
        self.exhibitions = self.load_exhibitions_data()
       
        # 시스템 종료 시그널 처리
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self.get_logger().info("MuseumGuideSystem node initialized.")

    def load_exhibitions_data(self):
        """data/exhibitions.json에서 전시품 정보 로드"""
        package_share_directory = get_package_share_directory('museum_guide_pkg')
        data_file = os.path.join(package_share_directory, 'data', "exhibitions.json")
       
        try:
            with open(data_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
           
            exhibitions = {}
            for id_str, exhibition in data["exhibitions"].items():
                exhibitions[int(id_str)] = {
                    "name": exhibition["name"],
                    "location": exhibition["location"],
                    "duration": exhibition["duration"],
                    "category": exhibition.get("category", "일반"),
                    "period": exhibition.get("period", ""),
                    "position": exhibition.get("position", {"x": 0, "y": 0, "theta": 0}),
                    "descriptions": exhibition.get("descriptions", {}),
                    "keywords": exhibition.get("keywords", []),}
           
            self.get_logger().info(f"{len(exhibitions)}개 전시품 정보를 JSON에서 로드했습니다.")
            return exhibitions
           
        except FileNotFoundError:
            self.get_logger().error(f"{data_file} 파일을 찾을 수 없습니다!")
            self.get_logger().error(f"💡 {data_file} 파일을 생성해주세요.")
            exit(1)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 파일 형식 오류: {str(e)}")
            self.get_logger().error(f"💡 {data_file} 파일의 JSON 형식을 확인해주세요.")
            exit(1)
        except Exception as e:
            self.get_logger().error(f"JSON 파일 로드 실패: {str(e)}")
            exit(1)

    def _start_ros_launch(self):
        """ROS2 Launch 파일을 백그라운드에서 실행"""
        if ROS2_AVAILABLE:
            launch_command = [
                "ros2", "launch", "museum_introducer_pkg", "museum_guide_launch.py"
            ]
            try:
                self.get_logger().info("ROS2 Launch 파일 실행 중...")
                # preexec_fn=os.setsid를 사용하여 새로운 프로세스 그룹 생성
                self.ros_launch_process = subprocess.Popen(
                    launch_command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid # 새로운 세션 리더로 만들어 프로세스 그룹 종료 가능하게 함
                )
                self.get_logger().info(f"ROS2 Launch 프로세스 시작 (PID: {self.ros_launch_process.pid})")
                # 잠시 대기하여 노드들이 초기화될 시간 부여
                time.sleep(3)
            except Exception as e:
                self.get_logger().error(f"ROS2 Launch 파일 실행 실패: {e}")
                self.ros_launch_process = None
        else:
            self.get_logger().warn("ROS2 환경이 아니므로 Launch 파일을 실행하지 않습니다.")

    def _stop_ros_launch(self):
        """실행 중인 ROS2 Launch 프로세스를 종료"""
        if self.ros_launch_process:
            self.get_logger().info("ROS2 Launch 프로세스 종료 중...")
            try:
                # 프로세스 그룹 전체에 SIGTERM 시그널 전송
                os.killpg(os.getpgid(self.ros_launch_process.pid), signal.SIGTERM)
                self.ros_launch_process.wait(timeout=5) # 5초 대기
                self.get_logger().info("ROS2 Launch 프로세스 종료 완료.")
            except ProcessLookupError:
                self.get_logger().warn("ROS2 Launch 프로세스가 이미 종료되었습니다.")
            except subprocess.TimeoutExpired:
                self.get_logger().error("ROS2 Launch 프로세스 종료 시간 초과. 강제 종료 시도...")
                os.killpg(os.getpgid(self.ros_launch_process.pid), signal.SIGKILL)
                self.get_logger().info("ROS2 Launch 프로세스 강제 종료 완료.")
            except Exception as e:
                self.get_logger().error(f"ROS2 Launch 프로세스 종료 실패: {e}")
            finally:
                self.ros_launch_process = None

    def signal_handler(self, signum, frame):
        """시스템 종료 시그널 처리"""
        self.get_logger().info("시스템 종료 신호를 받았습니다...")
        self.shutdown()

    def startup_sequence(self):
        """시스템 시작 시퀀스"""
        self.ui.show_startup_screen()
       
        # ROS2 Launch 파일 실행 (있는 경우에만)
        self._start_ros_launch()
       
        # ROS2 스피닝 시작 (있는 경우에만)
        if self.ros_manager:
            self.ros_manager.start_ros_spinning()
       
        # 터틀봇 연결 확인
        
       
        return True

    def main_loop(self):
        """메인 실행 루프"""
        try:
            # 시스템 시작
            if not self.startup_sequence():
                return
           
            while self.running:
                # 메인 메뉴 표시
                choice = self.ui.show_main_menu()
               
                if choice == "1":  # 새 관람 시작
                    self.start_new_tour()
                elif choice == "2":  # 전시품 정보 보기
                    self.show_exhibition_info()
                elif choice == "3":  # 시스템 상태 확인
                    self.show_system_status()
                elif choice == "4":  # 설정
                    self.show_settings()
                elif choice == "5":  # 프로필 관리
                    self.manage_profiles()
                elif choice == "0":  # 종료
                    self.shutdown()
                    break
                else:
                    self.ui.show_error("올바른 번호를 입력해주세요.")
                    time.sleep(1)
                   
        except KeyboardInterrupt:
            self.shutdown()
        except Exception as e:
            self.ui.show_error(f"시스템 오류가 발생했습니다: {str(e)}")
            self.shutdown()

    def start_new_tour(self):
        """새로운 관람 시작 - 단일 프로필 시스템 적용"""
        try:
            self.ui.show_message("새 관람을 시작합니다! 🎨")
           
            # 기존 사용자 선택 또는 새 사용자 등록
            if not self.user_profile.load_or_create_profile(self.ui):
                self.ui.show_message("사용자 설정을 취소했습니다.")
                return
           
            # 사용자 데이터를 ROS2로 전송 (가능한 경우)
            if self.ros_manager:
                self.ros_manager.send_user_profile(self.user_profile)
           
            # 관람 모드 선택
            mode = self.ui.select_tour_mode()
           
            if mode == "recommendation":
                self.start_recommendation_mode()
            elif mode == "tracking":
                self.start_tracking_mode()
            else:
                self.ui.show_message("관람을 취소했습니다.")
               
        except Exception as e:
            self.ui.show_error(f"관람 시작 중 오류: {str(e)}")
            print(f"Debug: {e}")

    def start_recommendation_mode(self):
        """추천 모드 시작"""
        try:
            self.ui.show_message("🤖 추천 모드를 시작합니다!")
           
            # 사용자 프로필 기반 전시품 추천
            recommended_exhibitions = self.get_recommendations()
           
            if not recommended_exhibitions:
                self.ui.show_error("추천할 전시품이 없습니다.")
                return
           
            self.ui.show_recommendations(recommended_exhibitions, self.exhibitions)
           
            # 사용자 확인
            if not self.ui.get_confirmation("추천 코스로 관람을 시작하시겠습니까?"):
                self.ui.show_message("추천 모드를 취소했습니다.")
                return
           
            # 추천 경로로 이동 시작
            for i, exhibit_id in enumerate(recommended_exhibitions, 1):
                if not self.running:
                    break
               
                exhibit = self.exhibitions[exhibit_id]
                self.ui.show_message(f"\n📍 {i}/{len(recommended_exhibitions)}번째 전시품으로 이동합니다")
                self.ui.show_current_destination(exhibit)
               
                # 터틀봇 이동
                if self.ros_manager:
                    target_position = exhibit.get("position", {"x": 0, "y": 0, "theta": 0})
                    self.ros_manager.send_navigation_goal(target_position)
                    # TODO: 네비게이션 완료 대기 로직 추가 필요
                else:
                    self.ui.show_error("ROS2 통합 모듈이 없어 터틀봇 이동을 할 수 없습니다.")
                    continue
               
                # 전시품 설명
                self.show_exhibition_details(exhibit_id)
               
                # 마지막 전시품이 아니면 계속 여부 확인
                if i < len(recommended_exhibitions):
                    if not self.ui.ask_continue_tour():
                        self.ui.show_message("관람을 중단합니다.")
                        break
           
            self.complete_tour()
           
        except Exception as e:
            self.ui.show_error(f"추천 모드 실행 중 오류: {str(e)}")
            print(f"Debug: {e}")

    def start_tracking_mode(self):
        """트래킹 모드 시작"""
        try:
            self.ui.show_message("👥 트래킹 모드를 시작합니다!")
           
            if self.ros_manager:
                # ROS2 연동 모드: 실제 QR코드 인식
                self.ui.show_message("QR코드가 있는 전시품 근처로 이동하시면 터틀봇이 따라갑니다.")
                self._tracking_mode_with_ros2()
            else:
                # 시뮬레이션 모드: 수동 입력
                self.ui.show_message("시뮬레이션 모드: 전시품 번호를 입력해주세요.")
                self._tracking_mode_simulation()
               
        except Exception as e:
            self.ui.show_error(f"트래킹 모드 실행 중 오류: {str(e)}")
            print(f"Debug: {e}")

    def _tracking_mode_with_ros2(self):
        """ROS2 연동 트래킹 모드"""
        visited_exhibitions = []
       
        while self.running:
            # 팀원들의 QR코드 인식 시스템에서 데이터 받기
            qr_detection = self.ros_manager.get_qr_detection()
           
            if qr_detection:
                exhibit_id = qr_detection.get('exhibition_id')
                if exhibit_id and 1 <= exhibit_id <= 7:
                    exhibit = self.exhibitions[exhibit_id]
                   
                    self.ui.show_detected_exhibition(exhibit)
                   
                    # 전시품 정보를 ROS2로 전송
                    self.ros_manager.send_exhibition_info(exhibit_id, exhibit)
                   
                    
                   
                    # 전시품 설명
                    self.show_exhibition_details(exhibit_id)
                   
                    if exhibit_id not in visited_exhibitions:
                        visited_exhibitions.append(exhibit_id)
                   
                    # 관람 계속할지 확인
                    if not self.ui.ask_continue_tracking():
                        if self.ros_manager:
                            self.ros_manager.send_robot_control_command("STOP")
                        break
            else:
                # QR코드가 감지되지 않으면 잠시 대기
                time.sleep(0.5)
       
        # 트래킹 모드 종료 시 로봇에게 SEARCH 명령 전송 (사용자가 명시적으로 종료하지 않은 경우)
        if self.ros_manager and self.running: # self.running이 True이면 사용자가 종료하지 않은 것
            self.ros_manager.send_robot_control_command("SEARCH")

        # 관람 완료
        self.user_profile.visited_exhibitions = visited_exhibitions
        self.complete_tour()

    def _tracking_mode_simulation(self):
        """시뮬레이션 트래킹 모드"""
        visited_exhibitions = []
       
        while self.running:
            # 스티커 인식 시뮬레이션
            user_input = self.ui.get_input("\n전시품 번호 입력 (1-7, 0:종료): ")
           
            if user_input == "0":
                break
           
            try:
                exhibit_id = int(user_input)
                if 1 <= exhibit_id <= 7:
                    exhibit = self.exhibitions[exhibit_id]
                   
                    self.ui.show_detected_exhibition(exhibit)
                   
                    
                   
                    # 전시품 설명
                    self.show_exhibition_details(exhibit_id)
                   
                    if exhibit_id not in visited_exhibitions:
                        visited_exhibitions.append(exhibit_id)
                   
                    # 관람 계속할지 확인
                    if not self.ui.ask_continue_tracking():
                        break
                else:
                    self.ui.show_error("1-7 사이의 번호를 입력해주세요.")
                   
            except ValueError:
                self.ui.show_error("올바른 숫자를 입력해주세요.")
       
        # 관람 완료
        self.user_profile.visited_exhibitions = visited_exhibitions
        self.complete_tour()

    def get_recommendations(self):
        """사용자 프로필 기반 전시품 추천"""
        # 간단한 추천 로직 (나중에 팀 C와 연동)
        interest = self.user_profile.interest_field
        level = self.user_profile.knowledge_level
        time_limit = self.user_profile.available_time
       
        # 시간에 따른 추천 개수 조정
        if time_limit <= 30:
            max_exhibitions = 2
        elif time_limit <= 60:
            max_exhibitions = 4
        else:
            max_exhibitions = 6
       
        # 관심 분야에 따른 추천
        recommendations = []
        if interest == "역사":
            recommendations = [1, 2, 3, 4]  # 고구려 벽화, 조선 백자, 불교 조각, 민속 생활용품
        elif interest == "예술":
            recommendations = [5, 6, 7, 1]  # 근현대 회화, 금속 공예품, 전통 의복, 고구려 벽화
        elif interest == "문화":
            recommendations = [4, 7, 2, 3]  # 민속 생활용품, 전통 의복, 조선 백자, 불교 조각
        else:  # 전체
            recommendations = [1, 2, 3, 4, 5, 6, 7]
       
        return recommendations[:max_exhibitions]

    def show_exhibition_details(self, exhibit_id):
        """전시품 상세 설명 표시""" 
        exhibit = self.exhibitions[exhibit_id]
       
        # 사용자 수준에 맞는 설명
        explanation = self.get_exhibition_explanation(exhibit_id)
       
        self.ui.show_exhibition_explanation(exhibit, explanation)
       
        # 관람 기록
        self.user_profile.add_visited_exhibition(exhibit_id)

    def get_exhibition_explanation(self, exhibit_id):
        """전시품 설명 생성 (JSON 데이터에서만 가져오기)"""
        level = self.user_profile.knowledge_level
       
        # JSON에서 로드한 전시품 정보 사용
        exhibition = self.exhibitions.get(exhibit_id)
        if not exhibition:
            return "이 전시품에 대한 정보를 찾을 수 없습니다."
       
        # JSON의 descriptions에서 설명 가져오기
        descriptions = exhibition.get("descriptions", {})
        if level in descriptions:
            return descriptions[level]
        elif "basic" in descriptions:
            # 해당 level이 없으면 basic을 기본으로 사용
            return descriptions["basic"]
        else:
            # descriptions가 전혀 없으면
            return f"{exhibition['name']}에 대한 상세 설명이 준비 중입니다."

    def show_exhibition_info(self):
        """전시품 정보 보기"""
        self.ui.show_exhibitions_list(self.exhibitions)
       
        while True:
            choice = self.ui.get_input("\n보고 싶은 전시품 번호 (1-7, 0:돌아가기): ")
           
            if choice == "0":
                break
            elif choice.isdigit() and 1 <= int(choice) <= 7:
                exhibit_id = int(choice)
                exhibit = self.exhibitions[exhibit_id]
                explanation = self.get_exhibition_explanation(exhibit_id)
                self.ui.show_exhibition_explanation(exhibit, explanation)
            else:
                self.ui.show_error("올바른 번호를 입력해주세요.")

    def show_system_status(self):
        """시스템 상태 표시"""
        turtlebot_status = "ROS2를 통해 터틀봇 상태 확인 필요"
        self.ui.show_system_status(turtlebot_status, self.user_profile)

    def show_settings(self):
        """설정 메뉴"""
        self.ui.show_settings_menu()

    def manage_profiles(self):
        """프로필 관리 메뉴"""
        while True:
            self.ui.clear_screen()
            self.ui.display_header("👤 프로필 관리")
           
            profiles = self.user_profile._get_existing_profiles()
           
            self.get_logger().info(f"총 등록된 사용자: {len(profiles)}명")
           
            if profiles:
                self.get_logger().info("등록된 사용자 목록:")
                for i, profile_info in enumerate(profiles, 1):
                    name = profile_info.get('name', 'Unknown')
                    user_id = profile_info['user_id']
                    last_visit = profile_info.get('last_visit', '방문 기록 없음')
                    self.get_logger().info(f"{i:2d}. {name:15} ({user_id}) - {last_visit}")
           
            self.get_logger().info("1. 프로필 상세 보기")
            self.get_logger().info("2. 프로필 삭제")
            self.get_logger().info("3. 중복 프로필 정리")
            self.get_logger().info("0. 돌아가기")
           
            choice = self.ui.get_input("\n선택: ")
           
            if choice == "0":
                break
            elif choice == "1":
                self._view_profile_details(profiles)
            elif choice == "2":
                self._delete_profile(profiles)
            elif choice == "3":
                self.user_profile.migrate_duplicate_profiles(self.get_logger())
                self.ui.show_success("중복 프로필 정리 완료!")
                input("계속하려면 엔터...")
            else:
                self.ui.show_error("올바른 번호를 선택해주세요.")

    def _view_profile_details(self, profiles):
        """프로필 상세 보기"""
        if not profiles:
            self.ui.show_error("등록된 프로필이 없습니다.")
            return
       
        self.get_logger().info("프로필을 선택하세요:")
        for i, profile_info in enumerate(profiles, 1):
            self.get_logger().info(f"{i}. {profile_info['name']} ({profile_info['user_id']})")
       
        try:
            choice = int(self.ui.get_input("번호 선택: "))
            if 1 <= choice <= len(profiles):
                selected_profile = profiles[choice - 1]
                self._show_profile_detail(selected_profile)
            else:
                self.ui.show_error("올바른 번호를 선택해주세요.")
        except ValueError:
            self.ui.show_error("숫자를 입력해주세요.")

    def _show_profile_detail(self, profile_info):
        """프로필 상세 정보 표시"""
        user_id = profile_info['user_id']
        filename = f"{user_id}.json"
        filepath = os.path.join("profiles", filename)
       
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
           
            self.ui.clear_screen()
            self.ui.display_header(f"👤 {data.get('name', 'Unknown')} 프로필 상세")
           
            self.get_logger().info(f"🆔 사용자 ID: {data.get('user_id', 'N/A')}")
            self.get_logger().info(f"👤 이름: {data.get('name', 'N/A')}")
            self.get_logger().info(f"🏷️ 닉네임: {data.get('nickname', 'N/A')}")
            self.get_logger().info(f"👥 연령대: {data.get('age_group', 'N/A')}")
            self.get_logger().info(f"🎨 관심 분야: {data.get('interest_field', 'N/A')}")
            self.get_logger().info(f"📚 지식 수준: {data.get('knowledge_level', 'N/A')}")
            self.get_logger().info(f"⏰ 선호 관람시간: {data.get('available_time', 'N/A')}분")
           
            visited = data.get('visited_exhibitions', [])
            self.get_logger().info(f"🎭 관람한 전시품: {len(visited)}개")
            if visited:
                self.get_logger().info(f"   → {', '.join(map(str, visited))}")
           
            self.get_logger().info(f"\n📅 프로필 생성: {data.get('profile_created', 'N/A')}")
            self.get_logger().info(f"🕐 마지막 방문: {data.get('last_visit', 'N/A')}")
           
            # 환경설정
            prefs = data.get('preferences', {})
            self.get_logger().info(f"\n⚙️ 환경설정:")
            self.get_logger().info(f"   언어: {prefs.get('language', 'N/A')}")
            self.get_logger().info(f"   음성 안내: {prefs.get('voice_guide', 'N/A')}")
            self.get_logger().info(f"   이동 속도: {prefs.get('walking_speed', 'N/A')}")
           
            input("\n계속하려면 엔터를 눌러주세요...")
           
        except Exception as e:
            self.ui.show_error(f"프로필 읽기 오류: {e}")

    def _delete_profile(self, profiles):
        """프로필 삭제"""
        if not profiles:
            self.ui.show_error("등록된 프로필이 없습니다.")
            return
       
        self.get_logger().info("삭제할 프로필을 선택하세요:")
        for i, profile_info in enumerate(profiles, 1):
            self.get_logger().info(f"{i}. {profile_info['name']} ({profile_info['user_id']})")
       
        try:
            choice = int(self.ui.get_input("번호 선택: "))
            if 1 <= choice <= len(profiles):
                selected_profile = profiles[choice - 1]
               
                # 확인
                confirm_msg = f"'{selected_profile['name']}' 프로필을 정말 삭제하시겠습니까?"
                if self.ui.get_confirmation(confirm_msg):
                    user_id = selected_profile['user_id']
                    filename = f"{user_id}.json"
                    filepath = os.path.join("profiles", filename)
                   
                    try:
                        os.remove(filepath)
                        self.ui.show_success(f"프로필이 삭제되었습니다: {selected_profile['name']}")
                    except Exception as e:
                        self.ui.show_error(f"삭제 실패: {e}")
                else:
                    self.ui.show_message("삭제를 취소했습니다.")
            else:
                self.ui.show_error("올바른 번호를 선택해주세요.")
        except ValueError:
            self.ui.show_error("숫자를 입력해주세요.")

    def complete_tour(self):
        """관람 완료 처리"""
        try:
            self.get_logger().info("🏁 === 관람 완료 처리 시작 ===")
           
            # end_time 설정 (generate_tour_report에서도 설정하지만 미리 설정)
            if not self.user_profile.end_time:
                self.user_profile.end_time = datetime.now()
           
            # 관람 통계 생성
            self.get_logger().info("📊 관람 통계 생성 중...")
            tour_stats = self.user_profile.generate_tour_report()
            self.get_logger().info(f"✅ 관람 통계 생성 완료 - 방문 전시품: {len(tour_stats['visited'])}개")
           
            # 최종 리포트 표시
            self.get_logger().info("📋 최종 리포트 표시 중...")
            self.ui.show_tour_report(tour_stats, self.exhibitions)
           
            # 사용자 데이터 저장
            self.get_logger().info("💾 === 사용자 프로필 저장 시작 ===")
            save_result = self.user_profile.save_profile()
           
            if save_result:
                self.get_logger().info("✅ 프로필 저장이 성공적으로 완료되었습니다!")
                self.ui.show_success("프로필이 성공적으로 저장되었습니다!")
            else:
                self.get_logger().error("❌ 프로필 저장에 실패했습니다.")
                self.ui.show_error("프로필 저장에 실패했습니다. 관리자에게 문의하세요.")
               
        except Exception as e:
            self.get_logger().error(f"❌ 관람 완료 처리 중 오류: {str(e)}")
            self.get_logger().error(f"🔍 오류 상세: {type(e).__name__}")
           
            # 그래도 프로필 저장은 시도
            self.get_logger().warn("🔄 오류에도 불구하고 프로필 저장을 시도합니다...")
            try:
                # 강제로 user_id 생성 (혹시 없을 경우)
                if not self.user_profile.user_id:
                    self.user_profile.user_id = f"emergency_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                    self.get_logger().info(f"🆘 긴급 user_id 생성: {self.user_profile.user_id}")
               
                self.user_profile.save_profile()
                self.get_logger().info("✅ 긴급 프로필 저장 성공!")
            except Exception as save_error:
                self.get_logger().error(f"❌ 긴급 프로필 저장도 실패: {str(save_error)}")
                # 최후의 수단: 현재 디렉토리에 간단한 백업 파일 생성
                try:
                    backup_data = {
                        "nickname": self.user_profile.nickname,
                        "visited_exhibitions": self.user_profile.visited_exhibitions,
                        "timestamp": datetime.now().isoformat()
                    }
                    with open(f"backup_profile_{datetime.now().strftime('%H%M%S')}.json", 'w') as f:
                        json.dump(backup_data, f, ensure_ascii=False, indent=2)
                    self.get_logger().info("📋 최소한의 백업 파일을 생성했습니다.")
                except:
                    self.get_logger().error("💥 모든 저장 시도가 실패했습니다.")
       
        self.get_logger().info("🏁 === 관람 완료 처리 종료 ===")

    def shutdown(self):
        """시스템 종료""" 
        self.running = False
        self.ui.show_shutdown_screen()
       
        # ROS2 시스템 종료 (있는 경우)
        if self.ros_manager:
            self.ros_manager.stop_ros()
       
        # ROS2 Launch 프로세스 종료 (있는 경우)
        self._stop_ros_launch()
       
        self.get_logger().info("박물관 가이드 시스템이 종료되었습니다.")


def main(args=None):
    """메인 함수 - 자동 마이그레이션 추가"""
    try:
        rclpy.init(args=args)
        node = MuseumGuideSystem()
        node.get_logger().info("🏛️ 박물관 스마트 전시 가이드 로봇 시스템")
        node.get_logger().info("=" * 50)
        migrate_old_structure(node.get_logger())
        node.get_logger().info("Starting main loop...")
        node.main_loop()
       
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    except Exception as e:
        node.get_logger().error(f"치명적 오류 발생: {str(e)}")
        node.get_logger().error("시스템을 종료합니다.")
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()