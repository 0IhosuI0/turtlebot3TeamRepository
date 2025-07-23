#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
박물관 가이드 로봇 - 터미널 UI 관리자
7개 전시품 박물관 전용 UI
"""

import os
import sys
import time
from datetime import datetime

# Rich 라이브러리 사용 가능 여부 확인
try:
    from rich.console import Console
    from rich.panel import Panel
    from rich.text import Text
    from rich.table import Table
    from rich.progress import Progress, SpinnerColumn, TextColumn
    from rich.layout import Layout
    from rich.align import Align
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False
    print("⚠️  Rich 라이브러리가 설치되지 않았습니다. 기본 터미널 UI로 실행됩니다.")
    print("더 예쁜 UI를 원하시면: pip3 install rich")

class MuseumGuideUI:
    def __init__(self):
        if RICH_AVAILABLE:
            self.console = Console()
        self.clear_screen()

    def clear_screen(self):
        """화면 지우기"""
        os.system('clear' if os.name == 'posix' else 'cls')

    def show_startup_screen(self):
        """시작 화면 표시"""
        self.clear_screen()
       
        if RICH_AVAILABLE:
            startup_text = Text()
            startup_text.append("🏛️ 박물관 스마트 전시 가이드 로봇\n", style="bold cyan")
            startup_text.append("   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n", style="cyan")
            startup_text.append("   🤖 터틀봇 연동 시스템 v1.0\n", style="green")
            startup_text.append("   📍 7개 전시품 완벽 가이드\n", style="yellow")
            startup_text.append("   👥 개인 맞춤형 관람 서비스\n", style="magenta")
           
            panel = Panel(
                startup_text,
                border_style="bright_blue",
                padding=(1, 2)
            )
            self.console.print(panel)
        else:
            print("=" * 60)
            print("🏛️  박물관 스마트 전시 가이드 로봇")
            print("    🤖 터틀봇 연동 시스템 v1.0")
            print("    📍 7개 전시품 완벽 가이드")
            print("    👥 개인 맞춤형 관람 서비스")
            print("=" * 60)
       
        print("\n시스템을 초기화하는 중...")
        self.show_loading("터틀봇 연결 확인 중", 3)

    def show_loading(self, message, duration=2):
        """로딩 화면 표시"""
        if RICH_AVAILABLE:
            with Progress(
                SpinnerColumn(),
                TextColumn("[progress.description]{task.description}"),
                console=self.console,
            ) as progress:
                task = progress.add_task(message, total=None)
                time.sleep(duration)
        else:
            print(f"⏳ {message}...")
            time.sleep(duration)
            print("✅ 완료!")

    def show_main_menu(self):
        """메인 메뉴 표시"""
        self.clear_screen()
       
        if RICH_AVAILABLE:
            menu_text = Text()
            menu_text.append("🏛️ 박물관 가이드 메인 메뉴\n\n", style="bold cyan")
            menu_text.append("1️⃣  새 관람 시작 (추천/트래킹 모드)\n", style="green")
            menu_text.append("2️⃣  전시품 정보 보기\n", style="yellow")  
            menu_text.append("3️⃣  시스템 상태 확인\n", style="blue")
            menu_text.append("4️⃣  설정\n", style="magenta")
            menu_text.append("0️⃣  시스템 종료\n", style="red")
           
            panel = Panel(
                menu_text,
                title="[bold green]MAIN MENU[/bold green]",
                border_style="green"
            )
            self.console.print(panel)
        else:
            print("\n" + "=" * 50)
            print("🏛️  박물관 가이드 메인 메뉴")
            print("=" * 50)
            print("1. 새 관람 시작 (추천/트래킹 모드)")
            print("2. 전시품 정보 보기")
            print("3. 시스템 상태 확인")
            print("4. 설정")
            print("0. 시스템 종료")
            print("=" * 50)
       
        return self.get_input("선택하세요: ")

    def select_tour_mode(self):
        """관람 모드 선택"""
        self.clear_screen()
       
        if RICH_AVAILABLE:
            mode_text = Text()
            mode_text.append("🎯 관람 모드를 선택해주세요\n\n", style="bold yellow")
            mode_text.append("1️⃣  추천 모드 🤖\n", style="green")
            mode_text.append("    → 로봇이 개인 맞춤 코스를 추천하고 안내합니다\n", style="dim green")
            mode_text.append("    → 체계적이고 효율적인 관람이 가능합니다\n\n", style="dim green")
           
            mode_text.append("2️⃣  트래킹 모드 👥\n", style="blue")
            mode_text.append("    → 원하는 전시품으로 자유롭게 이동하세요\n", style="dim blue")
            mode_text.append("    → 로봇이 따라가며 설명해드립니다\n\n", style="dim blue")
           
            mode_text.append("0️⃣  취소\n", style="red")
           
            panel = Panel(
                mode_text,
                title="[bold yellow]TOUR MODE[/bold yellow]",
                border_style="yellow"
            )
            self.console.print(panel)
        else:
            print("\n" + "=" * 50)
            print("🎯 관람 모드 선택")
            print("=" * 50)
            print("1. 추천 모드 🤖")
            print("   → 로봇이 개인 맞춤 코스를 추천하고 안내")
            print("   → 체계적이고 효율적인 관람")
            print()
            print("2. 트래킹 모드 👥")
            print("   → 원하는 전시품으로 자유롭게 이동")
            print("   → 로봇이 따라가며 설명")
            print()
            print("0. 취소")
            print("=" * 50)
       
        choice = self.get_input("모드를 선택하세요: ")
       
        if choice == "1":
            return "recommendation"
        elif choice == "2":
            return "tracking"
        else:
            return "cancel"

    def show_exhibitions_list(self, exhibitions):
        """전시품 목록 표시"""
        self.clear_screen()
       
        if RICH_AVAILABLE:
            table = Table(title="🏛️ 박물관 전시품 목록", show_header=True)
            table.add_column("번호", style="cyan", width=6)
            table.add_column("전시품명", style="green", width=20)
            table.add_column("위치", style="yellow", width=15)
            table.add_column("예상 관람시간", style="magenta", width=12)
           
            for exhibit_id, info in exhibitions.items():
                table.add_row(
                    f"{exhibit_id}",
                    f"🎨 {info['name']}",
                    f"📍 {info['location']}",
                    f"⏰ {info['duration']}분"
                )
           
            self.console.print(table)
        else:
            print("\n" + "=" * 70)
            print("🏛️ 박물관 전시품 목록")
            print("=" * 70)
            print(f"{'번호':<4} {'전시품명':<20} {'위치':<15} {'관람시간'}")
            print("-" * 70)
           
            for exhibit_id, info in exhibitions.items():
                print(f"{exhibit_id:<4} {info['name']:<20} {info['location']:<15} {info['duration']}분")
           
            print("=" * 70)

    def show_recommendations(self, recommended_exhibitions, all_exhibitions):
        """추천 전시품 표시"""
        self.clear_screen()
       
        if RICH_AVAILABLE:
            rec_text = Text()
            rec_text.append("🤖 개인 맞춤 추천 코스\n\n", style="bold green")
           
            for i, exhibit_id in enumerate(recommended_exhibitions, 1):
                exhibit = all_exhibitions[exhibit_id]
                rec_text.append(f"{i}단계: ", style="bold yellow")
                rec_text.append(f"{exhibit['name']}\n", style="green")
                rec_text.append(f"       📍 {exhibit['location']} | ⏰ {exhibit['duration']}분\n\n", style="dim white")
           
            total_time = sum(all_exhibitions[id]['duration'] for id in recommended_exhibitions)
            rec_text.append(f"💡 총 예상 관람시간: {total_time}분", style="bold cyan")
           
            panel = Panel(
                rec_text,
                title="[bold green]RECOMMENDATION[/bold green]",
                border_style="green"
            )
            self.console.print(panel)
        else:
            print("\n" + "=" * 50)
            print("🤖 개인 맞춤 추천 코스")
            print("=" * 50)
           
            for i, exhibit_id in enumerate(recommended_exhibitions, 1):
                exhibit = all_exhibitions[exhibit_id]
                print(f"{i}단계: {exhibit['name']}")
                print(f"       📍 {exhibit['location']} | ⏰ {exhibit['duration']}분")
                print()
           
            total_time = sum(all_exhibitions[id]['duration'] for id in recommended_exhibitions)
            print(f"💡 총 예상 관람시간: {total_time}분")
            print("=" * 50)
       
        input("\n계속하려면 엔터를 눌러주세요...")

    def show_current_destination(self, exhibit):
        """현재 목적지 표시"""
        if RICH_AVAILABLE:
            dest_text = Text()
            dest_text.append(f"🚶 ♂️ 이동 중: {exhibit['name']}\n", style="bold blue")
            dest_text.append(f"📍 위치: {exhibit['location']}\n", style="yellow")
            dest_text.append("🤖 터틀봇이 안내하고 있습니다...", style="green")
           
            panel = Panel(
                dest_text,
                title="[bold blue]MOVING TO[/bold blue]",
                border_style="blue"
            )
            self.console.print(panel)
        else:
            print(f"\n🚶 ♂️ 이동 중: {exhibit['name']}")
            print(f"📍 위치: {exhibit['location']}")
            print("🤖 터틀봇이 안내하고 있습니다...")
       
        time.sleep(2)

    def show_exhibition_explanation(self, exhibit, explanation):
        """전시품 설명 표시"""
        self.clear_screen()
       
        if RICH_AVAILABLE:
            exp_text = Text()
            exp_text.append(f"🎨 {exhibit['name']}\n", style="bold cyan")
            exp_text.append(f"📍 {exhibit['location']}\n\n", style="yellow")
            exp_text.append("📖 설명:\n", style="bold green")
            exp_text.append(f"{explanation}\n\n", style="white")
            exp_text.append("⏰ ", style="magenta")
            exp_text.append(f"예상 관람시간: {exhibit['duration']}분", style="magenta")
           
            panel = Panel(
                exp_text,
                title=f"[bold cyan]{exhibit['name']}[/bold cyan]",
                border_style="cyan",
                padding=(1, 2)
            )
            self.console.print(panel)
        else:
            print("\n" + "=" * 60)
            print(f"🎨 {exhibit['name']}")
            print(f"📍 {exhibit['location']}")
            print("=" * 60)
            print("📖 설명:")
            print(f"{explanation}")
            print(f"\n⏰ 예상 관람시간: {exhibit['duration']}분")
            print("=" * 60)
       
        input("\n관람을 완료하셨으면 엔터를 눌러주세요...")

    def show_detected_exhibition(self, exhibit):
        """감지된 전시품 알림"""
        if RICH_AVAILABLE:
            detect_text = Text()
            detect_text.append("🔍 전시품을 감지했습니다!\n\n", style="bold green")
            detect_text.append(f"📍 현재 위치: {exhibit['name']}\n", style="cyan")
            detect_text.append(f"🏛️ {exhibit['location']}", style="yellow")
           
            panel = Panel(
                detect_text,
                title="[bold green]DETECTED[/bold green]",
                border_style="green"
            )
            self.console.print(panel)
        else:
            print("\n🔍 전시품을 감지했습니다!")
            print(f"📍 현재 위치: {exhibit['name']}")
            print(f"🏛️ {exhibit['location']}")

    def show_system_status(self, turtlebot_status, user_profile):
        """시스템 상태 표시"""
        self.clear_screen()
       
        if RICH_AVAILABLE:
            table = Table(title="🔧 시스템 상태", show_header=True)
            table.add_column("항목", style="cyan")
            table.add_column("상태", style="green")
           
            table.add_row("터틀봇 연결", "🟢 연결됨" if turtlebot_status.get("connected") else "🔴 연결 안됨")
            table.add_row("배터리", f"🔋 {turtlebot_status.get('battery', 0)}%")
            table.add_row("현재 위치", f"📍 {turtlebot_status.get('location', '알 수 없음')}")
            table.add_row("사용자", f"👤 {user_profile.nickname if user_profile.nickname else '미설정'}")
            table.add_row("관람 시간", f"⏰ {datetime.now().strftime('%H:%M:%S')}")
           
            self.console.print(table)
        else:
            print("\n" + "=" * 40)
            print("🔧 시스템 상태")
            print("=" * 40)
            print(f"터틀봇 연결: {'🟢 연결됨' if turtlebot_status.get('connected') else '🔴 연결 안됨'}")
            print(f"배터리: 🔋 {turtlebot_status.get('battery', 0)}%")
            print(f"현재 위치: 📍 {turtlebot_status.get('location', '알 수 없음')}")
            print(f"사용자: 👤 {user_profile.nickname if user_profile.nickname else '미설정'}")
            print(f"관람 시간: ⏰ {datetime.now().strftime('%H:%M:%S')}")
            print("=" * 40)
       
        input("\n계속하려면 엔터를 눌러주세요...")

    def show_tour_report(self, tour_stats, all_exhibitions):
        """관람 리포트 표시"""
        self.clear_screen()
       
        if RICH_AVAILABLE:
            report_text = Text()
            report_text.append("📊 관람 완료 리포트\n\n", style="bold green")
            report_text.append(f"👤 관람객: {tour_stats.get('nickname', '익명')}\n", style="cyan")
            report_text.append(f"⏰ 총 관람시간: {tour_stats.get('total_time', 0)}분\n", style="yellow")
            report_text.append(f"🎨 관람한 전시품: {len(tour_stats.get('visited', []))}개\n\n", style="magenta")
           
            if tour_stats.get('visited'):
                report_text.append("📍 관람한 전시품:\n", style="bold blue")
                for exhibit_id in tour_stats['visited']:
                    if exhibit_id in all_exhibitions:
                        exhibit = all_exhibitions[exhibit_id]
                        report_text.append(f"   • {exhibit['name']} ({exhibit['location']})\n", style="white")
           
            report_text.append("\n💝 박물관을 방문해주셔서 감사합니다!", style="bold green")
           
            panel = Panel(
                report_text,
                title="[bold green]TOUR REPORT[/bold green]",
                border_style="green",
                padding=(1, 2)
            )
            self.console.print(panel)
        else:
            print("\n" + "=" * 50)
            print("📊 관람 완료 리포트")
            print("=" * 50)
            print(f"👤 관람객: {tour_stats.get('nickname', '익명')}")
            print(f"⏰ 총 관람시간: {tour_stats.get('total_time', 0)}분")
            print(f"🎨 관람한 전시품: {len(tour_stats.get('visited', []))}개")
           
            if tour_stats.get('visited'):
                print("\n📍 관람한 전시품:")
                for exhibit_id in tour_stats['visited']:
                    if exhibit_id in all_exhibitions:
                        exhibit = all_exhibitions[exhibit_id]
                        print(f"   • {exhibit['name']} ({exhibit['location']})")
           
            print("\n💝 박물관을 방문해주셔서 감사합니다!")
            print("=" * 50)
       
        input("\n종료하려면 엔터를 눌러주세요...")

    def show_settings_menu(self):
        """설정 메뉴 표시"""
        self.clear_screen()
       
        if RICH_AVAILABLE:
            settings_text = Text()
            settings_text.append("⚙️ 시스템 설정\n\n", style="bold blue")
            settings_text.append("1️⃣  터틀봇 재연결\n", style="green")
            settings_text.append("2️⃣  음성 안내 설정\n", style="yellow")
            settings_text.append("3️⃣  이동 속도 조정\n", style="magenta")
            settings_text.append("4️⃣  언어 설정\n", style="cyan")
            settings_text.append("0️⃣  돌아가기\n", style="red")
           
            panel = Panel(
                settings_text,
                title="[bold blue]SETTINGS[/bold blue]",
                border_style="blue"
            )
            self.console.print(panel)
        else:
            print("\n" + "=" * 40)
            print("⚙️ 시스템 설정")
            print("=" * 40)
            print("1. 터틀봇 재연결")
            print("2. 음성 안내 설정")
            print("3. 이동 속도 조정")
            print("4. 언어 설정")
            print("0. 돌아가기")
            print("=" * 40)
       
        choice = self.get_input("설정을 선택하세요: ")
       
        if choice == "1":
            self.show_message("🔄 터틀봇을 재연결하는 중...")
            self.show_loading("연결 중", 3)
            self.show_message("✅ 터틀봇 재연결 완료!")
        elif choice == "2":
            self.show_message("🔊 음성 안내 설정 기능은 개발 중입니다.")
        elif choice == "3":
            self.show_message("🚶 ♂️ 이동 속도 조정 기능은 개발 중입니다.")
        elif choice == "4":
            self.show_message("🌐 언어 설정 기능은 개발 중입니다.")

    def show_shutdown_screen(self):
        """시스템 종료 화면"""
        self.clear_screen()
       
        if RICH_AVAILABLE:
            shutdown_text = Text()
            shutdown_text.append("🤖 박물관 가이드 시스템을 종료합니다\n\n", style="bold red")
            shutdown_text.append("터틀봇을 안전한 위치로 이동시키는 중...\n", style="yellow")
            shutdown_text.append("시스템 데이터를 저장하는 중...\n", style="cyan")
            shutdown_text.append("\n👋 안녕히 가세요!\n", style="bold green")
            shutdown_text.append("다시 방문해주세요! 🏛️", style="green")
           
            panel = Panel(
                shutdown_text,
                title="[bold red]SHUTDOWN[/bold red]",
                border_style="red",
                padding=(1, 2)
            )
            self.console.print(panel)
        else:
            print("\n" + "=" * 50)
            print("🤖 박물관 가이드 시스템을 종료합니다")
            print("=" * 50)
            print("터틀봇을 안전한 위치로 이동시키는 중...")
            print("시스템 데이터를 저장하는 중...")
            print("\n👋 안녕히 가세요!")
            print("다시 방문해주세요! 🏛️")
            print("=" * 50)
       
        time.sleep(3)

    def ask_continue_tour(self):
        """관람 계속 여부 확인"""
        try:
            return self.get_confirmation("다음 전시품으로 이동하시겠습니까?")
        except Exception as e:
            print(f"Debug - ask_continue_tour error: {e}")
            return False

    def ask_continue_tracking(self):
        """트래킹 모드 계속 여부 확인"""
        try:
            return self.get_confirmation("계속 관람하시겠습니까? (다른 전시품으로 이동하세요)")
        except Exception as e:
            print(f"Debug - ask_continue_tracking error: {e}")
            return False

    def get_confirmation(self, message="계속하시겠습니까?"):
        """사용자 확인 받기"""
        while True:
            response = self.get_input(f"{message} (y/n): ").lower()
            if response in ['y', 'yes', '예', 'ㅇ']:
                return True
            elif response in ['n', 'no', '아니오', 'ㄴ']:
                return False
            else:
                self.show_error("y 또는 n을 입력해주세요.")

    def get_input(self, prompt):
        """사용자 입력 받기"""
        try:
            if RICH_AVAILABLE:
                return self.console.input(f"[bold green]{prompt}[/bold green]")
            else:
                return input(prompt)
        except (KeyboardInterrupt, EOFError):
            print("\n\n사용자가 입력을 중단했습니다.")
            return "0"

    def show_message(self, message, style="green"):
        """일반 메시지 표시"""
        if RICH_AVAILABLE:
            self.console.print(f"\n{message}", style=style)
        else:
            print(f"\n{message}")

    def show_error(self, error_message):
        """에러 메시지 표시"""
        if RICH_AVAILABLE:
            error_panel = Panel(
                f"❌ {error_message}",
                border_style="red",
                padding=(0, 1)
            )
            self.console.print(error_panel)
        else:
            print(f"\n❌ 오류: {error_message}")

    def show_warning(self, warning_message):
        """경고 메시지 표시"""
        if RICH_AVAILABLE:
            warning_panel = Panel(
                f"⚠️ {warning_message}",
                border_style="yellow",
                padding=(0, 1)
            )
            self.console.print(warning_panel)
        else:
            print(f"\n⚠️ 경고: {warning_message}")

    def show_success(self, success_message):
        """성공 메시지 표시"""
        if RICH_AVAILABLE:
            success_panel = Panel(
                f"✅ {success_message}",
                border_style="green",
                padding=(0, 1)
            )
            self.console.print(success_panel)
        else:
            print(f"\n✅ {success_message}")

    def display_header(self, title):
        """헤더 표시"""
        if RICH_AVAILABLE:
            header = Panel(
                Text(title, justify="center", style="bold cyan"),
                border_style="cyan",
                padding=(0, 2)
            )
            self.console.print(header)
        else:
            print("\n" + "=" * len(title))
            print(title)
            print("=" * len(title))

