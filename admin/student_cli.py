#!/usr/bin/env python3
import sys
import argparse
from student_management import StudentManager

def main():
    manager = StudentManager()
    parser = argparse.ArgumentParser(description='ROSForge Student Management')
    subparsers = parser.add_subparsers(dest='command', help='Commands')

    # Add student
    add_parser = subparsers.add_parser('add', help='Add new student')
    add_parser.add_argument('first_name', help='First name')
    add_parser.add_argument('last_name', help='Last name')
    add_parser.add_argument('--github', help='GitHub username')
    add_parser.add_argument('--gmail', help='Gmail address')
    add_parser.add_argument('--enrolled', help='Enrollment date (YYYY-MM-DD), default: today')
    add_parser.add_argument('--payment', default='unknown', choices=['cash', 'bank', 'transfer', 'other', 'unknown'], help='Payment method')
    add_parser.add_argument('--amount', type=float, default=0.0, help='Payment amount in TND')
    add_parser.add_argument('--notes', help='Additional notes')

    # Remove student (mark inactive)
    remove_parser = subparsers.add_parser('remove', help='Remove student access (keeps record)')
    remove_parser.add_argument('--github', help='GitHub username')
    remove_parser.add_argument('--gmail', help='Gmail address')

    # Delete student permanently
    delete_parser = subparsers.add_parser('delete', help='Permanently delete student record')
    delete_parser.add_argument('--github', help='GitHub username')
    delete_parser.add_argument('--gmail', help='Gmail address')

    # List students
    list_parser = subparsers.add_parser('list', help='List students')
    list_parser.add_argument('--status', choices=['active', 'inactive', 'all'], default='active', help='Filter by status')

    # Statistics
    subparsers.add_parser('stats', help='Show statistics')

    args = parser.parse_args()

    if args.command == 'add':
        if not args.github and not args.gmail:
            print("❌ Must provide --github or --gmail")
            sys.exit(1)
        success = manager.add_student(
            args.first_name, args.last_name,
            github_username=args.github, email=args.gmail,
            enrollment_date=args.enrolled,
            payment_method=args.payment, payment_amount=args.amount,
            notes=args.notes
        )
        if not success:
            sys.exit(1)

    elif args.command == 'remove':
        if not args.github and not args.gmail:
            print("❌ Must provide --github or --gmail")
            sys.exit(1)
        success = manager.remove_student(github_username=args.github, email=args.gmail)
        if not success:
            sys.exit(1)

    elif args.command == 'delete':
        if not args.github and not args.gmail:
            print("❌ Must provide --github or --gmail")
            sys.exit(1)
        success = manager.delete_student(github_username=args.github, email=args.gmail)
        if not success:
            sys.exit(1)

    elif args.command == 'list':
        students = manager.list_students(args.status)
        if students:
            print(f"\n📋 {args.status.title()} Students:")
            print(f"{'Name':<25} {'GitHub':<20} {'Gmail':<35} {'Enrolled':<12} {'Payment':<10} {'Amount (TND)'}")
            print("=" * 115)
            for s in students:
                name = f"{s['first_name']} {s['last_name']}"
                github = s['github_username'] or '-'
                email = s['email'] or '-'
                amount = f"{s['payment_amount']:.0f}" if s['payment_amount'] else '-'
                print(f"{name:<25} {github:<20} {email:<35} {s['enrollment_date']:<12} {s['payment_method']:<10} {amount}")
        else:
            print("No students found.")

    elif args.command == 'stats':
        stats = manager.get_statistics()
        print(f"\n📊 Cohort Statistics:")
        print(f"Total students:  {stats['total_students']}")
        print(f"Active students: {stats['active_students']}")
        print("\nPayment Methods:")
        for method, data in stats['payment_breakdown'].items():
            print(f"  {method}: {data['count']} students, {data['total_amount']:.0f} TND total")

    else:
        parser.print_help()

if __name__ == '__main__':
    main()
