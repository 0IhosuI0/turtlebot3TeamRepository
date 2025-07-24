#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
박물관 가이드 로봇 - 단일 프로필 시스템 (새 유물 카테고리 적용)
중복 제거: 하나의 사용자 = 하나의 JSON 파일
"""
import json
import time
import os
import shutil
from datetime import datetime

class UserProfile:
    def __init__(self):
        self.user_id = ""
        self.name = ""
        self.nickname = ""
        self.interest_field = ""
        self.age_group = ""
        self.knowledge_level = ""
        self.available_time = 0
        self.visited_exhibitions = []
        self.start_time = None
        self.end_time = None
        self.preferences = {
            "language": "ko",
            "voice_guide": True,
            "walking_speed": "normal"
        }
        self.profiles_dir = "profiles"
        self._ensure_directory()

    def _ensure_directory(self):
        if not os.path.exists(self.profiles_dir):
            os.makedirs(self.profiles_dir, exist_ok=True)

    def load_or_create_profile(self, ui):
        existing_profiles = self._get_existing_profiles()
        if existing_profiles:
            ui.clear_screen()
            ui.display_header("👤 사용자 선택")
            print("\n📋 기존 사용자:")
            for i, profile_info in enumerate(existing_profiles, 1):
                name = profile_info.get('name', profile_info.get('nickname', 'Unknown'))
                user_id = profile_info['user_id']
                last_visit = profile_info.get('last_visit', '첫 방문')
                print(f"{i}. {name} ({user_id}) - 마지막 방문: {last_visit}")
            print(f"{len(existing_profiles) + 1}. 새 사용자 등록")
            while True:
                choice = ui.get_input(f"\n선택하세요 (1-{len(existing_profiles) + 1}): ")
                try:
                    choice_num = int(choice)
                    if 1 <= choice_num <= len(existing_profiles):
                        selected_profile = existing_profiles[choice_num - 1]
                        if self._load_existing_profile(selected_profile['user_id']):
                            self.start_time = datetime.now()
                            ui.show_success(f"환영합니다, {self.nickname}님! 🎉")
                            return True
                        else:
                            ui.show_error("프로필 로드 실패")
                            return False
                    elif choice_num == len(existing_profiles) + 1:
                        return self.setup_new_profile(ui)
                    else:
                        ui.show_error("올바른 번호를 선택해주세요.")
                except ValueError:
                    ui.show_error("숫자를 입력해주세요.")
        else:
            ui.show_message("첫 번째 사용자입니다. 새 프로필을 생성합니다.")
            return self.setup_new_profile(ui)

    def _get_existing_profiles(self):
        profiles = []
        if not os.path.exists(self.profiles_dir):
            return profiles
        for filename in os.listdir(self.profiles_dir):
            if filename.endswith('.json'):
                try:
                    filepath = os.path.join(self.profiles_dir, filename)
                    with open(filepath, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    profiles.append({
                        'user_id': data.get('user_id', filename[:-5]),
                        'name': data.get('name', data.get('nickname', 'Unknown')),
                        'nickname': data.get('nickname', data.get('name', 'Unknown')),
                        'last_visit': data.get('last_visit', '첫 방문'),
                        'filename': filename
                    })
                except Exception as e:
                    print(f"⚠️ 프로필 읽기 오류 {filename}: {e}")
                    continue
        return profiles

    def _load_existing_profile(self, user_id):
        possible_files = [
            f"{user_id}.json",
            f"profile_{user_id}.json",
            f"visitor_{user_id}.json"
        ]
        for filename in possible_files:
            filepath = os.path.join(self.profiles_dir, filename)
            if os.path.exists(filepath):
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    self.user_id = data.get('user_id', user_id)
                    self.name = data.get('name', '')
                    self.nickname = data.get('nickname', data.get('name', ''))
                    self.age_group = data.get('age_group', self._age_to_age_group(data.get('age', 25)))
                    self.interest_field = self._normalize_interest(data.get('interest_field', data.get('interests', [])))
                    self.knowledge_level = data.get('knowledge_level', 'intermediate')
                    self.available_time = data.get('available_time', 60)
                    self.visited_exhibitions = data.get('visited_exhibitions', data.get('visit_history', []))
                    self.preferences = data.get('preferences', self.preferences)
                    data['last_visit'] = datetime.now().isoformat()
                    with open(filepath, 'w', encoding='utf-8') as f:
                        json.dump(data, f, ensure_ascii=False, indent=2)
                    print(f"✅ 기존 프로필 로드: {self.nickname}")
                    return True
                except Exception as e:
                    print(f"❌ 프로필 로드 오류: {e}")
                    continue
        print(f"❌ 프로필을 찾을 수 없습니다: {user_id}")
        return False

    def _normalize_interest(self, interest_data):
        if isinstance(interest_data, str):
            return interest_data
        elif isinstance(interest_data, list) and interest_data:
            mapping = {
                "craft": "공예",
                "ceramics": "도자",
                "metal": "금속/종교",
                "history": "공예",
                "art": "도자",
                "religion": "금속/종교"
            }
            first_interest = str(interest_data[0]).lower()
            return mapping.get(first_interest, "전체")
        else:
            return "전체"

    def _age_to_age_group(self, age):
        if age < 14:
            return "어린이"
        elif age < 20:
            return "청소년"
        elif age < 40:
            return "청년"
        elif age < 60:
            return "중년"
        else:
            return "시니어"

    def setup_new_profile(self, ui):
        ui.clear_screen()
        ui.display_header("👤 새 사용자 등록")
        try:
            while True:
                self.nickname = ui.get_input("\n닉네임을 입력해주세요: ").strip()
                if self.nickname:
                    self.name = self.nickname
                    break
                ui.show_error("닉네임을 입력해주세요.")
            self.user_id = f"visitor_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            self.age_group = self._select_age_group(ui)
            if not self.age_group:
                return False
            self.interest_field = self._select_interest_field(ui)
            if not self.interest_field:
                return False
            self.knowledge_level = self._select_knowledge_level(ui)
            if not self.knowledge_level:
                return False
            self.available_time = self._select_available_time(ui)
            if not self.available_time:
                return False
            if self._confirm_profile(ui):
                self.start_time = datetime.now()
                ui.show_success(f"환영합니다, {self.nickname}님! 🎉")
                return True
            else:
                return self.setup_new_profile(ui)
        except KeyboardInterrupt:
            ui.show_message("설정을 취소했습니다.")
            return False

    def save_profile(self):
        if not self.user_id:
            self.user_id = f"visitor_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        filename = f"{self.user_id}.json"
        filepath = os.path.join(self.profiles_dir, filename)
        profile_data = {
            "user_id": self.user_id,
            "name": self.name,
            "nickname": self.nickname,
            "age_group": self.age_group,
            "interest_field": self.interest_field,
            "interests": [self._interest_to_english(self.interest_field)],
            "knowledge_level": self.knowledge_level,
            "available_time": self.available_time,
            "visited_exhibitions": self.visited_exhibitions,
            "visit_history": self.visited_exhibitions,
            "preferences": self.preferences,
            "start_time": self.start_time.isoformat() if self.start_time else None,
            "end_time": self.end_time.isoformat() if self.end_time else None,
            "profile_created": datetime.now().isoformat(),
            "last_visit": datetime.now().isoformat(),
            "system_info": {
                "profile_type": "unified",
                "created_by": "museum_guide_system",
                "last_updated": datetime.now().isoformat()
            }
        }
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(profile_data, f, ensure_ascii=False, indent=2)
            print(f"✅ 프로필 저장 완료: {filepath}")
            return True
        except Exception as e:
            print(f"❌ 프로필 저장 실패: {e}")
            return False

    def _interest_to_english(self, korean_interest):
        mapping = {
            "공예": "craft",
            "도자": "ceramics",
            "금속/종교": "metal",
            "전체": "general"
        }
        return mapping.get(korean_interest, "general")

    def migrate_duplicate_profiles(self):
        print("🔄 중복 프로필 정리 시작...")
        users_dir = "users"
        if os.path.exists(users_dir):
            for filename in os.listdir(users_dir):
                if filename.endswith('.json'):
                    old_path = os.path.join(users_dir, filename)
                    new_path = os.path.join(self.profiles_dir, filename)
                    if not os.path.exists(new_path):
                        shutil.move(old_path, new_path)
                        print(f"📁 이동: {filename}")
            try:
                if not os.listdir(users_dir):
                    os.rmdir(users_dir)
                    print("🗑️ 빈 users/ 디렉토리 제거")
            except:
                pass
        print("✅ 중복 프로필 정리 완료")

    def _select_age_group(self, ui):
        ui.clear_screen()
        ui.display_header("👥 연령대 선택")
        age_groups = {
            "1": "어린이 (7-13세)",
            "2": "청소년 (14-19세)",
            "3": "청년 (20-39세)",
            "4": "중년 (40-59세)",
            "5": "시니어 (60세 이상)"
        }
        print("\n연령대를 선택해주세요:")
        for key, value in age_groups.items():
            print(f"{key}. {value}")
        print("0. 이전으로")
        while True:
            choice = ui.get_input("\n선택: ")
            if choice == "0":
                return None
            elif choice in age_groups:
                selected = age_groups[choice].split(" ")[0]
                ui.show_success(f"{selected} 연령대로 설정되었습니다.")
                return selected
            else:
                ui.show_error("올바른 번호를 선택해주세요.")

    def _select_interest_field(self, ui):
        ui.clear_screen()
        ui.display_header("🎨 관심 분야 선택")
        # 최신 유물 분류 및 예시 적용
        interests = {
            "1": "공예 (나전칠기함, 동종, 청동연꽃모양 연적, 청동주전자)",
            "2": "도자 (백자병, 청자 잔 받침)",
            "3": "금속/종교 (향로, 동종, 청동주전자)",
            "4": "전체 (모든 분야에 관심)"
        }
        print("\n관심 있는 분야를 선택해주세요:")
        for key, value in interests.items():
            print(f"{key}. {value}")
        print("0. 이전으로")
        while True:
            choice = ui.get_input("\n선택: ")
            if choice == "0":
                return None
            elif choice in interests:
                field_map = {"1": "공예", "2": "도자", "3": "금속/종교", "4": "전체"}
                selected = field_map[choice]
                ui.show_success(f"{selected} 분야로 설정되었습니다.")
                return selected
            else:
                ui.show_error("올바른 번호를 선택해주세요.")

    def _select_knowledge_level(self, ui):
        ui.clear_screen()
        ui.display_header("📚 지식 수준 선택")
        levels = {
            "1": "기초 - 쉽고 간단한 설명 원함",
            "2": "중급 - 적당한 수준의 전문 지식 원함",
            "3": "고급 - 상세하고 전문적인 설명 원함"
        }
        print("\n박물관/문화재에 대한 지식 수준을 선택해주세요:")
        for key, value in levels.items():
            print(f"{key}. {value}")
        print("0. 이전으로")
        while True:
            choice = ui.get_input("\n선택: ")
            if choice == "0":
                return None
            elif choice in levels:
                level_map = {"1": "basic", "2": "intermediate", "3": "advanced"}
                selected = level_map[choice]
                level_names = {"basic": "기초", "intermediate": "중급", "advanced": "고급"}
                ui.show_success(f"{level_names[selected]} 수준으로 설정되었습니다.")
                return selected
            else:
                ui.show_error("올바른 번호를 선택해주세요.")

    def _select_available_time(self, ui):
        ui.clear_screen()
        ui.display_header("⏰ 관람 시간 선택")
        time_options = {
            "1": "30분 - 빠른 관람 (2-3개 전시품)",
            "2": "60분 - 기본 관람 (4-5개 전시품)",
            "3": "90분 - 여유 관람 (6-7개 전시품)",
            "4": "직접 입력"
        }
        print("\n관람 가능한 시간을 선택해주세요:")
        for key, value in time_options.items():
            print(f"{key}. {value}")
        print("0. 이전으로")
        while True:
            choice = ui.get_input("\n선택: ")
            if choice == "0":
                return None
            elif choice == "1":
                ui.show_success("30분 관람으로 설정되었습니다.")
                return 30
            elif choice == "2":
                ui.show_success("60분 관람으로 설정되었습니다.")
                return 60
            elif choice == "3":
                ui.show_success("90분 관람으로 설정되었습니다.")
                return 90
            elif choice == "4":
                while True:
                    try:
                        custom_time = int(ui.get_input("관람 시간을 분 단위로 입력하세요 (10-180): "))
                        if 10 <= custom_time <= 180:
                            ui.show_success(f"{custom_time}분 관람으로 설정되었습니다.")
                            return custom_time
                        else:
                            ui.show_error("10분에서 180분 사이로 입력해주세요.")
                    except ValueError:
                        ui.show_error("숫자를 입력해주세요.")
            else:
                ui.show_error("올바른 번호를 선택해주세요.")

    def _confirm_profile(self, ui):
        ui.clear_screen()
        ui.display_header("✅ 설정 확인")
        print(f"\n👤 닉네임: {self.nickname}")
        print(f"👥 연령대: {self.age_group}")
        print(f"🎨 관심 분야: {self.interest_field}")
        print(f"📚 지식 수준: {self.knowledge_level}")
        print(f"⏰ 관람 시간: {self.available_time}분")
        return ui.get_confirmation("\n이 설정으로 관람을 시작하시겠습니까?")

    def add_visited_exhibition(self, exhibit_id):
        if exhibit_id not in self.visited_exhibitions:
            self.visited_exhibitions.append(exhibit_id)

    def generate_tour_report(self):
        self.end_time = datetime.now()
        if self.start_time:
            actual_time = int((self.end_time - self.start_time).total_seconds() / 60)
        else:
            actual_time = 0
        return {
            "visitor_id": self.user_id,
            "nickname": self.nickname,
            "age_group": self.age_group,
            "interest_field": self.interest_field,
            "knowledge_level": self.knowledge_level,
            "planned_time": self.available_time,
            "actual_time": actual_time,
            "visited": self.visited_exhibitions,
            "total_exhibitions": len(self.visited_exhibitions),
            "completion_rate": len(self.visited_exhibitions) / 7 * 100,
            "start_time": self.start_time.strftime("%Y-%m-%d %H:%M:%S") if self.start_time else None,
            "end_time": self.end_time.strftime("%Y-%m-%d %H:%M:%S") if self.end_time else None
        }


    # 호환성 메서드들
    def setup_profile(self, ui):
        """main.py 호환성을 위한 메서드"""
        return self.load_or_create_profile(ui)
   
    @property
    def visitor_id(self):
        """main.py 호환성을 위한 속성"""
        return self.user_id
   
    def get_recommendation_preference(self):
        """추천을 위한 선호도 정보 반환"""
        return {
            "interest_field": self.interest_field,
            "knowledge_level": self.knowledge_level,
            "available_time": self.available_time,
            "age_group": self.age_group
        }

    def update_preferences(self, new_preferences):
        """사용자 환경설정 업데이트"""
        self.preferences.update(new_preferences)

    def get_visiting_progress(self):
        """관람 진행 상황 반환"""
        total_exhibitions = 7
        visited_count = len(self.visited_exhibitions)
        progress_percentage = (visited_count / total_exhibitions) * 100
       
        return {
            "total": total_exhibitions,
            "visited": visited_count,
            "remaining": total_exhibitions - visited_count,
            "progress": round(progress_percentage, 1)
        }
